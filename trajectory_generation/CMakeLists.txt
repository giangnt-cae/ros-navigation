cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generation)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_generation
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(trajectory_generation src/trajectory_generation.cpp
  src/quintic_bezier_splines.cpp
  src/velocity_profile.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(trajectory_generation
  ${catkin_LIBRARIES}
  ${Eigen_INCLUDE_DRIS}
)

