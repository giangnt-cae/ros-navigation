cmake_minimum_required(VERSION 3.0.2)
project(vk_costmap_2d)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  message_filters
  message_generation
  laser_geometry
)

add_message_files(
  FILES
  AgvInfo.msg
  AgvInfoArray.msg
)

add_service_files(
  FILES
  GetAgvInfo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES vk_costmap_2d_lib
 CATKIN_DEPENDS geometry_msgs
                nav_msgs
                roscpp
                rospy 
                sensor_msgs 
                std_msgs 
                tf tf2 
                tf2_msgs 
                tf2_ros 
                tf2_geometry_msgs
                message_runtime
                laser_geometry
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vk_costmap_2d_lib
  src/layer.cpp
  src/costmap_2d.cpp
  src/costmap_layer.cpp
  src/layered_costmap.cpp
  src/static_layer.cpp
  src/inflation_layer.cpp
  src/footprint.cpp
  src/agv_layer.cpp
  src/observation.cpp
  src/obstacle_layer.cpp
)

add_executable(vk_costmap_2d src/costmap_2d_ros.cpp)

target_link_libraries(vk_costmap_2d
  vk_costmap_2d_lib
  ${catkin_LIBRARIES}
)

add_executable(test_node src/test_node.cpp
)

target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

