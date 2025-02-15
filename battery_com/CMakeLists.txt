cmake_minimum_required(VERSION 3.0.2)
project(battery_com)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(CppLinuxSerial REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES battery_com
 CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(battery_com src/battery_com.cpp)

target_link_libraries(battery_com ${catkin_LIBRARIES})
target_link_libraries(battery_com CppLinuxSerial::CppLinuxSerial)
