cmake_minimum_required(VERSION 3.0.2)
project(convert_polygon)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  message_filters
  message_generation
)

add_message_files(
  FILES
  ObstacleMsg.msg
  ObstacleArrayMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES convert_polygon_lib
 CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2
    tf2_msgs
    tf2_ros
    tf2_geometry_msgs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(convert_polygon_lib
  src/convert_to_polygons.cpp
)

add_executable(convert_polygon_node src/convert_to_polygons_node.cpp)

target_link_libraries(convert_polygon_node
  convert_polygon_lib
  ${catkin_LIBRARIES}
)
