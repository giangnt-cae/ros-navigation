cmake_minimum_required(VERSION 3.0.2)
project(vk_global_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vk_global_planner
  CATKIN_DEPENDS geometry_msgs message_filters nav_msgs roscpp sensor_msgs tf tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vk_global_planner src/main.cpp
  src/rrt_star.cpp
)

target_link_libraries(vk_global_planner
  ${catkin_LIBRARIES}
)
