cmake_minimum_required(VERSION 3.0.2)
project(graph_based_slam)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_based_slam
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf tf2 tf2_msgs tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
add_executable(graph_based_slam src/graph_based_slam.cpp
  src/map.cpp
  src/scan_matcher.cpp
  src/optimization.cpp
)

target_link_libraries(graph_based_slam
  ${catkin_LIBRARIES}
  ${Eigen_INCLUDE_DRIS}
)

