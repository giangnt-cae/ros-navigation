cmake_minimum_required(VERSION 3.0.2)
project(vk_slam_3d)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vk_slam_3d
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(vk_slam_3d src/vk_slam_3d.cpp
  src/voxel_grid.cpp
  src/voxel_map.cpp
  src/scan_matcher.cpp
  src/odometry_model.cpp
  src/graphbasedslam.cpp
)


target_link_libraries(vk_slam_3d
  ${catkin_LIBRARIES}
  ${Eigen_INCLUDE_DRIS} Boost::thread
)