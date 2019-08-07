cmake_minimum_required(VERSION 2.8.3)
project(avoidance_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs
  tf2_sensor_msgs
)

find_package(PCL REQUIRED
  common
)

add_service_files(
  FILES
  avoidance_server.srv
  avoidance_move_server.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES avoidance_server
  CATKIN_DEPENDS std_msgs message_runtime
# DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(
    avoidance_server_node
    src/avoidance_server_node.cpp
    src/avoidance_server.cpp
    include/avoidance_server.h
)
target_link_libraries(avoidance_server_node
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
)
