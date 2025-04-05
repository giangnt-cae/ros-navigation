cmake_minimum_required(VERSION 3.0.2)
project(communication)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  vk_costmap_2d
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES communication
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2 vk_costmap_2d
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(agv_client src/client.cpp)
add_executable(agv_server src/server.cpp)

target_link_libraries(agv_client ${catkin_LIBRARIES})
target_link_libraries(agv_server ${catkin_LIBRARIES})

